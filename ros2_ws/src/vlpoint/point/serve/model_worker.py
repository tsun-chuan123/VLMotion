"""
A model worker executes the model.
"""
import argparse
import asyncio
import contextlib
import json
import re
import time
import threading
import uuid

from fastapi import FastAPI, Request, BackgroundTasks
from fastapi.responses import StreamingResponse
import requests
import torch
import uvicorn
from functools import partial
import string

from point.constants import WORKER_HEART_BEAT_INTERVAL
from point.utils import (build_logger, server_error_msg,
    pretty_print_semaphore)
from point.model.builder import load_pretrained_model
from point.mm_utils import process_images, load_image_from_base64, tokenizer_image_token
from point.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN, DEFAULT_IM_START_TOKEN, DEFAULT_IM_END_TOKEN
from transformers import TextIteratorStreamer
from threading import Thread


GB = 1 << 30

worker_id = str(uuid.uuid4())[:6]
logger = build_logger("model_worker", f"model_worker_{worker_id}.log")
global_counter = 0

model_semaphore = None


def str2bool(value):
    if isinstance(value, bool):
        return value
    value = value.strip().lower()
    if value in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def parse_coords(text: str):
    coords = []
    for match in re.findall(r'\(\s*([\d.]+)\s*,\s*([\d.]+)\s*\)', text):
        coords.append((float(match[0]), float(match[1])))
    return coords


def parse_candidate_labels(text: str, valid_labels):
    valid = set(valid_labels)
    labels = []
    for match in re.findall(r'\b([A-Z]|P\d+)\b', text.upper()):
        if match in valid and match not in labels:
            labels.append(match)
    return labels


def snap_coords_to_candidates(coords, detections, max_points):
    labels = []
    for coord in coords:
        best = min(
            detections,
            key=lambda det: (det["center"][0] - coord[0]) ** 2 + (det["center"][1] - coord[1]) ** 2,
        )
        if best["label"] not in labels:
            labels.append(best["label"])
        if max_points > 0 and len(labels) >= max_points:
            break
    return labels


def resolve_dtype(dtype_name, device):
    dtype_name = str(dtype_name or "auto").lower()
    if dtype_name in ("auto", ""):
        return torch.float32 if torch.device(device).type == "cpu" else torch.bfloat16
    if dtype_name in ("fp32", "float32"):
        return torch.float32
    if dtype_name in ("bf16", "bfloat16"):
        return torch.bfloat16
    if dtype_name in ("fp16", "float16", "half"):
        return torch.float16
    raise ValueError(f"Unsupported dtype: {dtype_name}")


def detect_sam3_candidates(img, args):
    if not args.sam3_detect_buttons:
        return []

    from transformers import Sam3Model, Sam3Processor

    tower_name = getattr(args, "sam3_vision_tower", None) or getattr(args, "mm_sam3_vision_tower", None) or "facebook/sam3"

    device = torch.device(args.sam3_detect_device)
    dtype = resolve_dtype(args.sam3_detect_dtype, device)
    print(
        "SAM3 button detector: "
        f"text={args.sam3_detect_text!r}, "
        f"threshold={args.sam3_detect_threshold}, "
        f"device={device}, "
        f"dtype={dtype}"
    )

    processor = Sam3Processor.from_pretrained(tower_name)
    model = Sam3Model.from_pretrained(tower_name)
    model.requires_grad_(False)
    model.eval()
    model.to(device=device, dtype=dtype)

    inputs = processor(images=img, text=args.sam3_detect_text, return_tensors="pt")
    inputs = {
        key: value.to(device=device) if hasattr(value, "to") else value
        for key, value in inputs.items()
    }
    if "pixel_values" in inputs:
        inputs["pixel_values"] = inputs["pixel_values"].to(dtype=dtype)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_object_detection(
        outputs,
        threshold=args.sam3_detect_threshold,
        target_sizes=[img.size[::-1]],
    )[0]

    boxes = results["boxes"].detach().float().cpu()
    scores = results["scores"].detach().float().cpu()
    order = torch.argsort(scores, descending=True)

    detections = []
    labels = list(string.ascii_uppercase)
    width, height = img.size
    for rank, idx in enumerate(order[:args.sam3_max_candidates]):
        x1, y1, x2, y2 = boxes[idx].tolist()
        score = float(scores[idx].item())
        detections.append({
            "label": labels[rank] if rank < len(labels) else f"P{rank + 1}",
            "score": score,
            "box": (x1, y1, x2, y2),
            "center": (((x1 + x2) / 2.0) / width, ((y1 + y2) / 2.0) / height),
        })

    print(f"SAM3 detected {len(detections)} candidate button(s).")
    for det in detections:
        cx, cy = det["center"]
        x1, y1, x2, y2 = det["box"]
        print(
            f"  {det['label']}: score={det['score']:.3f}, "
            f"center=({cx:.3f}, {cy:.3f}), "
            f"box=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})"
        )

    del model
    if device.type == "cuda":
        torch.cuda.empty_cache()

    return detections


def heart_beat_worker(controller):

    while True:
        time.sleep(WORKER_HEART_BEAT_INTERVAL)
        controller.send_heart_beat()


class ModelWorker:
    def __init__(self, controller_addr, worker_addr,
                 worker_id, no_register,
                 model_path, model_base, model_name,
                 load_8bit, load_4bit, device, use_flash_attn=False,
                 mm_use_sam3_conditioning=False,
                 mm_sam3_vision_tower=None,
                 mm_sam3_blend_alpha=0.35,
                 mm_sam3_mask_gamma=1.0,
                 mm_sam3_device="cpu",
                 mm_sam3_dtype="auto",
                 mm_sam3_unload_after_forward=False):
        self.controller_addr = controller_addr
        self.worker_addr = worker_addr
        self.worker_id = worker_id
        if model_path.endswith("/"):
            model_path = model_path[:-1]
        if model_name is None:
            model_paths = model_path.split("/")
            if model_paths[-1].startswith('checkpoint-'):
                self.model_name = model_paths[-2] + "_" + model_paths[-1]
            else:
                self.model_name = model_paths[-1]
        else:
            self.model_name = model_name

        self.device = device
        logger.info(f"Loading the model {self.model_name} on worker {worker_id} ...")
        load_start = time.time()
        self.tokenizer, self.model, self.image_processor, self.context_len = load_pretrained_model(
            model_path, model_base, self.model_name, load_8bit, load_4bit, device=self.device, use_flash_attn=use_flash_attn)
        load_elapsed = time.time() - load_start
        logger.info(
            "Model load completed in "
            f"{load_elapsed:.2f}s ({load_elapsed / 60.0:.2f} min)"
        )
        self._apply_sam3_runtime_config(
            mm_use_sam3_conditioning=mm_use_sam3_conditioning,
            mm_sam3_vision_tower=mm_sam3_vision_tower,
            mm_sam3_blend_alpha=mm_sam3_blend_alpha,
            mm_sam3_mask_gamma=mm_sam3_mask_gamma,
            mm_sam3_device=mm_sam3_device,
            mm_sam3_dtype=mm_sam3_dtype,
            mm_sam3_unload_after_forward=mm_sam3_unload_after_forward,
        )

    def _apply_sam3_runtime_config(self, mm_use_sam3_conditioning,
                                   mm_sam3_vision_tower,
                                   mm_sam3_blend_alpha,
                                   mm_sam3_mask_gamma,
                                   mm_sam3_device,
                                   mm_sam3_dtype,
                                   mm_sam3_unload_after_forward):
        mm_use_sam3_conditioning = bool(mm_use_sam3_conditioning)
        mm_sam3_blend_alpha = float(mm_sam3_blend_alpha)
        mm_sam3_mask_gamma = float(mm_sam3_mask_gamma)
        mm_sam3_vision_tower = mm_sam3_vision_tower or None

        if mm_use_sam3_conditioning and not mm_sam3_vision_tower:
            raise ValueError(
                "--mm-sam3-vision-tower is required when "
                "--mm-use-sam3-conditioning is true."
            )

        self.model.config.mm_use_sam3_conditioning = mm_use_sam3_conditioning
        self.model.config.mm_sam3_vision_tower = mm_sam3_vision_tower
        self.model.config.mm_sam3_blend_alpha = mm_sam3_blend_alpha
        self.model.config.mm_sam3_mask_gamma = mm_sam3_mask_gamma
        self.model.config.mm_sam3_device = mm_sam3_device
        self.model.config.mm_sam3_dtype = mm_sam3_dtype
        self.model.config.mm_sam3_unload_after_forward = mm_sam3_unload_after_forward

        vision_tower = self.model.get_vision_tower()
        if vision_tower is not None:
            vision_tower.mm_use_sam3_conditioning = mm_use_sam3_conditioning
            vision_tower.mm_sam3_vision_tower = mm_sam3_vision_tower
            vision_tower.mm_sam3_blend_alpha = mm_sam3_blend_alpha
            vision_tower.mm_sam3_mask_gamma = mm_sam3_mask_gamma
            vision_tower.mm_sam3_device = mm_sam3_device
            vision_tower.mm_sam3_dtype = mm_sam3_dtype
            vision_tower.mm_sam3_unload_after_forward = mm_sam3_unload_after_forward

        logger.info(
            "SAM3 conditioning config: "
            f"enabled={mm_use_sam3_conditioning}, "
            f"tower={mm_sam3_vision_tower}, "
            f"alpha={mm_sam3_blend_alpha}, "
            f"gamma={mm_sam3_mask_gamma}, "
            f"device={mm_sam3_device}, "
            f"dtype={mm_sam3_dtype}, "
            f"unload_after_forward={mm_sam3_unload_after_forward}"
        )

    def register_to_controller(self):
        logger.info("Register to controller")

        url = self.controller_addr + "/register_worker"
        data = {
            "worker_name": self.worker_addr,
            "check_heart_beat": True,
            "worker_status": self.get_status()
        }
        r = requests.post(url, json=data)
        assert r.status_code == 200

    def send_heart_beat(self):
        logger.info(f"Send heart beat. Models: {[self.model_name]}. "
                    f"Semaphore: {pretty_print_semaphore(model_semaphore)}. "
                    f"global_counter: {global_counter}")

        url = self.controller_addr + "/receive_heart_beat"

        while True:
            try:
                ret = requests.post(url, json={
                    "worker_name": self.worker_addr,
                    "queue_length": self.get_queue_length()}, timeout=5)
                exist = ret.json()["exist"]
                break
            except requests.exceptions.RequestException as e:
                logger.error(f"heart beat error: {e}")
            time.sleep(5)

        if not exist:
            self.register_to_controller()

    def get_queue_length(self):
        if model_semaphore is None:
            return 0
        else:
            return args.limit_model_concurrency - model_semaphore._value + (len(
                model_semaphore._waiters) if model_semaphore._waiters is not None else 0)

    def get_status(self):
        return {
            "model_names": [self.model_name],
            "speed": 1,
            "queue_length": self.get_queue_length(),
        }

    @torch.inference_mode()
    def generate_stream(self, params):
        tokenizer, model, image_processor = self.tokenizer, self.model, self.image_processor
        lm_head = getattr(model, "lm_head", None)
        amp_dtype = None
        if lm_head is not None and hasattr(lm_head, "weight"):
            if lm_head.weight.dtype in (torch.float16, torch.bfloat16):
                amp_dtype = lm_head.weight.dtype

        prompt = params["prompt"]
        ori_prompt = prompt
        images = params.get("images", None)
        num_image_tokens = 0
        sam3_candidates = []
        if images is not None and len(images) > 0:
            if len(images) > 0:
                if len(images) != prompt.count(DEFAULT_IMAGE_TOKEN):
                    raise ValueError("Number of images does not match number of <image> tokens in prompt")

                pil_images = [load_image_from_base64(image) for image in images]
                image_sizes = [image.size for image in pil_images]
                if pil_images:
                    sam3_candidates = detect_sam3_candidates(pil_images[0], args)
                images = process_images(pil_images, image_processor, model.config)

                image_dtype = amp_dtype if amp_dtype is not None else torch.float16
                if type(images) is list:
                    images = [image.to(self.model.device, dtype=image_dtype) for image in images]
                else:
                    images = images.to(self.model.device, dtype=image_dtype)

                replace_token = DEFAULT_IMAGE_TOKEN
                if getattr(self.model.config, 'mm_use_im_start_end', False):
                    replace_token = DEFAULT_IM_START_TOKEN + replace_token + DEFAULT_IM_END_TOKEN
                prompt = prompt.replace(DEFAULT_IMAGE_TOKEN, replace_token)

                if sam3_candidates:
                    candidate_lines = []
                    for det in sam3_candidates:
                        cx, cy = det["center"]
                        candidate_lines.append(
                            f"{det['label']}: center=({cx:.3f}, {cy:.3f}), score={det['score']:.3f}"
                        )
                    prompt += (
                        "\nSAM3 found these candidate buttons:\n"
                        + "\n".join(candidate_lines)
                        + "\nYou must choose only from the SAM3 candidate labels above. "
                        "Do not invent a new coordinate."
                    )

                num_image_tokens = prompt.count(replace_token) * model.get_vision_tower().num_patches
            else:
                images = None
                image_sizes = None
            image_args = {"images": images, "image_sizes": image_sizes}
        else:
            images = None
            image_args = {}

        temperature = float(params.get("temperature", 1.0))
        top_p = float(params.get("top_p", 1.0))
        max_context_length = getattr(model.config, 'max_position_embeddings', 2048)
        max_new_tokens = min(int(params.get("max_new_tokens", 256)), 1024)
        use_cache = str2bool(str(params.get("use_cache", False)))
        stop_str = params.get("stop", None)
        do_sample = True if temperature > 0.001 else False

        input_ids = tokenizer_image_token(prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors='pt').unsqueeze(0).to(self.device)
        keywords = [stop_str]
        # stopping_criteria = KeywordsStoppingCriteria(keywords, tokenizer, input_ids)
        streamer = TextIteratorStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True, timeout=15)

        max_new_tokens = min(max_new_tokens, max_context_length - input_ids.shape[-1] - num_image_tokens)

        if max_new_tokens < 1:
            yield json.dumps({"text": ori_prompt + "Exceeds max token length. Please start a new conversation, thanks.", "error_code": 0}).encode() + b"\0"
            return

        generation_kwargs = dict(
            inputs=input_ids,
            do_sample=do_sample,
            max_new_tokens=max_new_tokens,
            streamer=streamer,
            use_cache=use_cache,
            **image_args
        )
        if do_sample:
            generation_kwargs["temperature"] = temperature
            generation_kwargs["top_p"] = top_p

        generation_error = []

        def run_generate():
            try:
                autocast_ctx = contextlib.nullcontext()
                if torch.device(self.device).type == "cuda" and amp_dtype is not None:
                    autocast_ctx = torch.autocast(device_type="cuda", dtype=amp_dtype)
                with torch.no_grad(), autocast_ctx:
                    model.generate(**generation_kwargs)
            except RuntimeError as exc:
                if (
                    "expected mat1 and mat2 to have the same dtype" in str(exc)
                    and lm_head is not None
                    and hasattr(lm_head, "to")
                ):
                    print("Detected dtype mismatch in lm_head; retrying with lm_head cast to float32...")
                    try:
                        lm_head.to(dtype=torch.float32)
                        if torch.cuda.is_available():
                            torch.cuda.empty_cache()
                        with torch.no_grad():
                            model.generate(**generation_kwargs)
                        return
                    except Exception as retry_exc:
                        generation_error.append(retry_exc)
                else:
                    generation_error.append(exc)
                streamer.end()
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            except Exception as exc:
                generation_error.append(exc)
                streamer.end()
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()

        thread = Thread(target=run_generate)
        thread.start()

        generated_text = ori_prompt
        for new_text in streamer:
            generated_text += new_text
            if stop_str and generated_text.endswith(stop_str):
                generated_text = generated_text[:-len(stop_str)]
        thread.join()
        if generation_error:
            raise generation_error[0]

        completion_text = generated_text[len(ori_prompt):]

        if sam3_candidates:
            label_to_coord = {det["label"]: det["center"] for det in sam3_candidates}
            selected_labels = parse_candidate_labels(generated_text, label_to_coord.keys())
            if not selected_labels:
                generated_coords = parse_coords(generated_text)
                selected_labels = snap_coords_to_candidates(
                    generated_coords,
                    sam3_candidates,
                    args.sam3_max_points,
                )
                if selected_labels:
                    print(f"Snapped generated coordinates to nearest SAM3 labels: {selected_labels}")
            if not selected_labels:
                fallback_count = args.sam3_max_points if args.sam3_max_points > 0 else len(sam3_candidates)
                selected_labels = [det["label"] for det in sam3_candidates[:fallback_count]]
                print(f"No usable model selection; falling back to top SAM3 labels: {selected_labels}")
            if args.sam3_max_points > 0:
                selected_labels = selected_labels[:args.sam3_max_points]
            coords = [label_to_coord[label] for label in selected_labels]
            completion_text = str([(round(nx, 4), round(ny, 4)) for nx, ny in coords])
            print(f"Selected SAM3 labels: {selected_labels}")

        yield json.dumps({"text": ori_prompt + completion_text, "error_code": 0}).encode() + b"\0"

    def generate_stream_gate(self, params):
        try:
            for x in self.generate_stream(params):
                yield x
        except ValueError as e:
            print("Caught ValueError:", e)
            ret = {
                "text": server_error_msg,
                "error_code": 1,
            }
            yield json.dumps(ret).encode() + b"\0"
        except torch.cuda.CudaError as e:
            print("Caught torch.cuda.CudaError:", e)
            ret = {
                "text": server_error_msg,
                "error_code": 1,
            }
            yield json.dumps(ret).encode() + b"\0"
        except Exception as e:
            print("Caught Unknown Error", e)
            ret = {
                "text": server_error_msg,
                "error_code": 1,
            }
            yield json.dumps(ret).encode() + b"\0"


app = FastAPI()


def release_model_semaphore(fn=None):
    model_semaphore.release()
    if fn is not None:
        fn()


@app.post("/worker_generate_stream")
async def generate_stream(request: Request):
    global model_semaphore, global_counter
    global_counter += 1
    params = await request.json()

    if model_semaphore is None:
        model_semaphore = asyncio.Semaphore(args.limit_model_concurrency)
    await model_semaphore.acquire()
    worker.send_heart_beat()
    generator = worker.generate_stream_gate(params)
    background_tasks = BackgroundTasks()
    background_tasks.add_task(partial(release_model_semaphore, fn=worker.send_heart_beat))
    return StreamingResponse(generator, background=background_tasks)


@app.post("/worker_get_status")
async def get_status(request: Request):
    return worker.get_status()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=22000)
    parser.add_argument("--worker-address", type=str,
        default="http://10.0.0.1:22000")
    parser.add_argument("--controller-address", type=str,
        default="http://10.0.0.1:11000")
    parser.add_argument("--model-path", type=str, default="PME033541/vla13")
    parser.add_argument("--model-base", type=str, default=None)
    parser.add_argument("--model-name", type=str)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--multi-modal", action="store_true", help="Multimodal mode is automatically detected with model name, please make sure `llava` is included in the model path.")
    parser.add_argument("--limit-model-concurrency", type=int, default=5)
    parser.add_argument("--stream-interval", type=int, default=1)
    parser.add_argument("--no-register", action="store_true")
    parser.add_argument("--load-8bit", action="store_true")
    parser.add_argument("--load-4bit", action="store_true")
    parser.add_argument("--use-flash-attn", action="store_true")
    parser.add_argument("--mm-use-sam3-conditioning", type=str2bool, default=True)
    parser.add_argument("--mm-sam3-vision-tower", type=str, default="facebook/sam3")
    parser.add_argument("--mm-sam3-blend-alpha", type=float, default=0.35)
    parser.add_argument("--mm-sam3-mask-gamma", type=float, default=1.0)
    parser.add_argument("--mm-sam3-device", type=str, default="cuda")
    parser.add_argument("--mm-sam3-dtype", type=str, default="bfloat16")
    parser.add_argument("--mm-sam3-unload-after-forward", type=str2bool, default=False)
    parser.add_argument("--sam3-detect-buttons", type=str2bool, default=True)
    parser.add_argument("--sam3-detect-text", type=str, default="button")
    parser.add_argument("--sam3-detect-threshold", type=float, default=0.25)
    parser.add_argument("--sam3-max-candidates", type=int, default=8)
    parser.add_argument("--sam3-max-points", type=int, default=4)
    parser.add_argument("--sam3-detect-device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--sam3-detect-dtype", type=str, default="bfloat16")
    args = parser.parse_args()
    logger.info(f"args: {args}")

    if args.multi_modal:
        logger.warning("Multimodal mode is automatically detected with model name, please make sure `llava` is included in the model path.")

    worker = ModelWorker(args.controller_address,
                         args.worker_address,
                         worker_id,
                         args.no_register,
                         args.model_path,
                         args.model_base,
                         args.model_name,
                         args.load_8bit,
                         args.load_4bit,
                         args.device,
                         use_flash_attn=args.use_flash_attn,
                         mm_use_sam3_conditioning=args.mm_use_sam3_conditioning,
                         mm_sam3_vision_tower=args.mm_sam3_vision_tower,
                         mm_sam3_blend_alpha=args.mm_sam3_blend_alpha,
                         mm_sam3_mask_gamma=args.mm_sam3_mask_gamma,
                         mm_sam3_device=args.mm_sam3_device,
                         mm_sam3_dtype=args.mm_sam3_dtype,
                         mm_sam3_unload_after_forward=args.mm_sam3_unload_after_forward)
    if not args.no_register:
        worker.register_to_controller()
        heart_beat_thread = threading.Thread(
            target=heart_beat_worker, args=(worker,), daemon=True)
        heart_beat_thread.start()
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
