import torch
import torch.nn as nn
import warnings

from transformers import CLIPVisionModel, CLIPImageProcessor, CLIPVisionConfig

try:
    import cv2
except ImportError:
    cv2 = None


class CLIPVisionTower(nn.Module):
    def __init__(self, vision_tower, args, delay_load=False):
        super().__init__()

        self.is_loaded = False

        self.vision_tower_name = vision_tower
        self.select_layer = args.mm_vision_select_layer
        self.select_feature = getattr(args, 'mm_vision_select_feature', 'patch')
        self.mm_use_canny_edge = getattr(args, 'mm_use_canny_edge', False)
        self.mm_canny_low_threshold = int(getattr(args, 'mm_canny_low_threshold', 100))
        self.mm_canny_high_threshold = int(getattr(args, 'mm_canny_high_threshold', 200))
        self.mm_canny_blend_alpha = float(getattr(args, 'mm_canny_blend_alpha', 0.2))

        if self.mm_canny_low_threshold >= self.mm_canny_high_threshold:
            warnings.warn(
                "mm_canny_low_threshold should be smaller than mm_canny_high_threshold. "
                "Swapping values automatically."
            )
            self.mm_canny_low_threshold, self.mm_canny_high_threshold = (
                self.mm_canny_high_threshold,
                self.mm_canny_low_threshold,
            )

        if not delay_load:
            self.load_model()
        elif getattr(args, 'unfreeze_mm_vision_tower', False):
            self.load_model()
        else:
            self.cfg_only = CLIPVisionConfig.from_pretrained(self.vision_tower_name)

    def _apply_canny_edge_conditioning(self, images: torch.Tensor) -> torch.Tensor:
        if (not self.mm_use_canny_edge) or images.ndim != 4 or images.shape[1] != 3:
            return images

        if cv2 is None:
            raise ImportError(
                "OpenCV is required when mm_use_canny_edge=True. "
                "Please install opencv-python-headless."
            )

        input_dtype = images.dtype
        images_fp32 = images.to(dtype=torch.float32)

        mean = torch.tensor(self.image_processor.image_mean, device=images_fp32.device, dtype=images_fp32.dtype).view(1, 3, 1, 1)
        std = torch.tensor(self.image_processor.image_std, device=images_fp32.device, dtype=images_fp32.dtype).view(1, 3, 1, 1)

        # CLIPImageProcessor outputs normalized tensors, so recover RGB in [0, 1] before Canny.
        rgb_images = (images_fp32 * std + mean).clamp(0.0, 1.0)
        cpu_uint8 = (rgb_images.detach().cpu() * 255.0).round().to(dtype=torch.uint8)

        fused = []
        alpha = self.mm_canny_blend_alpha
        for idx in range(cpu_uint8.shape[0]):
            rgb_hwc = cpu_uint8[idx].permute(1, 2, 0).numpy()
            gray = cv2.cvtColor(rgb_hwc, cv2.COLOR_RGB2GRAY)
            edges = cv2.Canny(gray, self.mm_canny_low_threshold, self.mm_canny_high_threshold)
            edge_map = torch.from_numpy(edges).to(device=images_fp32.device, dtype=images_fp32.dtype) / 255.0
            edge_rgb = edge_map.unsqueeze(0).repeat(3, 1, 1)
            fused.append((1.0 - alpha) * rgb_images[idx] + alpha * edge_rgb)

        fused_rgb = torch.stack(fused, dim=0)
        fused_normalized = (fused_rgb - mean) / std
        return fused_normalized.to(dtype=input_dtype)

    def load_model(self, device_map=None):
        if self.is_loaded:
            print('{} is already loaded, `load_model` called again, skipping.'.format(self.vision_tower_name))
            return

        self.image_processor = CLIPImageProcessor.from_pretrained(self.vision_tower_name)
        self.vision_tower = CLIPVisionModel.from_pretrained(self.vision_tower_name, device_map=device_map)
        self.vision_tower.requires_grad_(False)

        self.is_loaded = True

    def feature_select(self, image_forward_outs):
        image_features = image_forward_outs.hidden_states[self.select_layer]
        if self.select_feature == 'patch':
            image_features = image_features[:, 1:]
        elif self.select_feature == 'cls_patch':
            image_features = image_features
        else:
            raise ValueError(f'Unexpected select feature: {self.select_feature}')
        return image_features

    @torch.no_grad()
    def forward(self, images):
        if type(images) is list:
            image_features = []
            for image in images:
                image_input = self._apply_canny_edge_conditioning(image.unsqueeze(0))
                image_forward_out = self.vision_tower(image_input.to(device=self.device, dtype=self.dtype), output_hidden_states=True)
                image_feature = self.feature_select(image_forward_out).to(image.dtype)
                image_features.append(image_feature)
        else:
            image_input = self._apply_canny_edge_conditioning(images)
            image_forward_outs = self.vision_tower(image_input.to(device=self.device, dtype=self.dtype), output_hidden_states=True)
            image_features = self.feature_select(image_forward_outs).to(images.dtype)

        return image_features

    @property
    def dummy_feature(self):
        return torch.zeros(1, self.hidden_size, device=self.device, dtype=self.dtype)

    @property
    def dtype(self):
        return self.vision_tower.dtype

    @property
    def device(self):
        return self.vision_tower.device

    @property
    def config(self):
        if self.is_loaded:
            return self.vision_tower.config
        else:
            return self.cfg_only

    @property
    def hidden_size(self):
        return self.config.hidden_size

    @property
    def num_patches_per_side(self):
        return self.config.image_size // self.config.patch_size

    @property
    def num_patches(self):
        return (self.config.image_size // self.config.patch_size) ** 2



class CLIPVisionTowerS2(CLIPVisionTower):
    def __init__(self, vision_tower, args, delay_load=False):
        super().__init__(vision_tower, args, delay_load)

        self.s2_scales = getattr(args, 's2_scales', '336,672,1008')
        self.s2_scales = list(map(int, self.s2_scales.split(',')))
        self.s2_scales.sort()
        self.s2_split_size = self.s2_scales[0]
        self.s2_image_size = self.s2_scales[-1]

        try:
            from s2wrapper import forward as multiscale_forward
        except ImportError:
            raise ImportError('Package s2wrapper not found! Please install by running: \npip install git+https://github.com/bfshi/scaling_on_scales.git')
        self.multiscale_forward = multiscale_forward

        # change resize/crop size in preprocessing to the largest image size in s2_scale
        if not delay_load or getattr(args, 'unfreeze_mm_vision_tower', False):
            self.image_processor.size['shortest_edge'] = self.s2_image_size
            self.image_processor.crop_size['height'] = self.image_processor.crop_size['width'] = self.s2_image_size

    def load_model(self, device_map=None):
        if self.is_loaded:
            print('{} is already loaded, `load_model` called again, skipping.'.format(self.vision_tower_name))
            return

        self.image_processor = CLIPImageProcessor.from_pretrained(self.vision_tower_name)
        self.vision_tower = CLIPVisionModel.from_pretrained(self.vision_tower_name, device_map=device_map)
        self.vision_tower.requires_grad_(False)

        self.image_processor.size['shortest_edge'] = self.s2_image_size
        self.image_processor.crop_size['height'] = self.image_processor.crop_size['width'] = self.s2_image_size

        self.is_loaded = True

    @torch.no_grad()
    def forward_feature(self, images):
        image_input = self._apply_canny_edge_conditioning(images)
        image_forward_outs = self.vision_tower(image_input.to(device=self.device, dtype=self.dtype), output_hidden_states=True)
        image_features = self.feature_select(image_forward_outs).to(images.dtype)
        return image_features

    @torch.no_grad()
    def forward(self, images):
        if type(images) is list:
            image_features = []
            for image in images:
                image_feature = self.multiscale_forward(self.forward_feature, image.unsqueeze(0), img_sizes=self.s2_scales, max_split_size=self.s2_split_size)
                image_features.append(image_feature)
        else:
            image_features = self.multiscale_forward(self.forward_feature, images, img_sizes=self.s2_scales, max_split_size=self.s2_split_size)

        return image_features

    @property
    def hidden_size(self):
        return self.config.hidden_size * len(self.s2_scales)
