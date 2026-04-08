import datetime
import logging
import logging.handlers
import os
import re
import sys
import warnings

import requests

from point.constants import LOGDIR

server_error_msg = "**NETWORK ERROR DUE TO HIGH TRAFFIC. PLEASE REGENERATE OR REFRESH THIS PAGE.**"
moderation_msg = "YOUR INPUT VIOLATES OUR CONTENT MODERATION GUIDELINES. PLEASE TRY AGAIN."

handler = None

_NOISY_LINE_PATTERNS = [
    re.compile(r"HTTP Request:\\s"),
    re.compile(r"LOAD REPORT from:"),
    re.compile(r"\\|\\s+UNEXPECTED\\s+\\|"),
    re.compile(r"^Notes:$"),
    re.compile(r"^-\\s+UNEXPECTED:"),
    re.compile(r"The following generation flags are not valid and may be ignored"),
]


def _should_suppress_log_line(line: str) -> bool:
    text = line.strip()
    if not text:
        return True
    return any(pattern.search(text) for pattern in _NOISY_LINE_PATTERNS)


def _configure_third_party_logging():
    """Reduce verbose dependency logs in worker output."""
    # Keep hub/model logs quiet unless there is a real warning/error.
    os.environ.setdefault("HF_HUB_VERBOSITY", "error")
    os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")

    # Suppress noisy informational HTTP traces and model load summaries.
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("huggingface_hub").setLevel(logging.ERROR)
    logging.getLogger("transformers").setLevel(logging.ERROR)

    # Keep uvicorn lifecycle logs visible (startup/shutdown/bind address).
    uvicorn_logger = logging.getLogger("uvicorn")
    uvicorn_logger.setLevel(logging.INFO)
    uvicorn_logger.propagate = True
    uvicorn_error_logger = logging.getLogger("uvicorn.error")
    uvicorn_error_logger.setLevel(logging.INFO)
    uvicorn_error_logger.propagate = True

    try:
        from transformers.utils import logging as transformers_logging
        transformers_logging.set_verbosity_error()
    except Exception:
        pass

    warnings.filterwarnings(
        "ignore",
        message=r".*_check_is_size will be removed in a future PyTorch release.*",
        category=FutureWarning,
    )


def build_logger(logger_name, logger_filename):
    global handler

    formatter = logging.Formatter(
        fmt="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Set the format of root handlers
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.INFO)
    logging.getLogger().handlers[0].setFormatter(formatter)

    _configure_third_party_logging()

    # Redirect stdout and stderr to loggers
    stdout_logger = logging.getLogger("stdout")
    stdout_logger.setLevel(logging.INFO)
    sl = StreamToLogger(stdout_logger, logging.INFO)
    sys.stdout = sl

    stderr_logger = logging.getLogger("stderr")
    # Keep stderr channel open to INFO so remapped uvicorn INFO lines are visible.
    stderr_logger.setLevel(logging.INFO)
    sl = StreamToLogger(stderr_logger, logging.ERROR)
    sys.stderr = sl

    # Get logger
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.INFO)

    # Add a file handler for all loggers
    if handler is None:
        os.makedirs(LOGDIR, exist_ok=True)
        filename = os.path.join(LOGDIR, logger_filename)
        handler = logging.handlers.TimedRotatingFileHandler(
            filename, when='D', utc=True, encoding='UTF-8')
        handler.setFormatter(formatter)

        for name, item in logging.root.manager.loggerDict.items():
            if isinstance(item, logging.Logger):
                item.addHandler(handler)

    return logger


class StreamToLogger(object):
    """
    Fake file-like stream object that redirects writes to a logger instance.
    """
    def __init__(self, logger, log_level=logging.INFO):
        self.terminal = sys.stdout
        self.logger = logger
        self.log_level = log_level
        self.linebuf = ''

    def __getattr__(self, attr):
        return getattr(self.terminal, attr)

    def write(self, buf):
        # tqdm/HF progress bars update with '\r' instead of '\n'.
        # Treat carriage returns as line breaks so progress updates are visible.
        temp_linebuf = (self.linebuf + buf).replace('\r', '\n')
        self.linebuf = ''
        for line in temp_linebuf.splitlines(True):
            # From the io.TextIOWrapper docs:
            #   On output, if newline is None, any '\n' characters written
            #   are translated to the system default line separator.
            # By default sys.stdout.write() expects '\n' newlines and then
            # translates them so this is still cross platform.
            if line[-1] == '\n':
                if not _should_suppress_log_line(line):
                    level = self.log_level
                    # uvicorn often writes INFO lines to stderr; keep severity meaningful.
                    if self.log_level >= logging.ERROR and line.lstrip().startswith("INFO:"):
                        level = logging.INFO
                    self.logger.log(level, line.rstrip())
            else:
                # Keep partial stderr output visible to avoid losing fatal errors at process exit.
                if self.log_level >= logging.ERROR:
                    if not _should_suppress_log_line(line):
                        level = self.log_level
                        if line.lstrip().startswith("INFO:"):
                            level = logging.INFO
                        self.logger.log(level, line.rstrip())
                else:
                    self.linebuf += line

    def flush(self):
        if self.linebuf != '':
            if not _should_suppress_log_line(self.linebuf):
                level = self.log_level
                if self.log_level >= logging.ERROR and self.linebuf.lstrip().startswith("INFO:"):
                    level = logging.INFO
                self.logger.log(level, self.linebuf.rstrip())
        self.linebuf = ''


def disable_torch_init():
    """
    Disable the redundant torch default initialization to accelerate model creation.
    """
    import torch
    setattr(torch.nn.Linear, "reset_parameters", lambda self: None)
    setattr(torch.nn.LayerNorm, "reset_parameters", lambda self: None)


def violates_moderation(text):
    """
    Check whether the text violates OpenAI moderation API.
    """
    url = "https://api.openai.com/v1/moderations"
    headers = {"Content-Type": "application/json",
               "Authorization": "Bearer " + os.environ["OPENAI_API_KEY"]}
    text = text.replace("\n", "")
    data = "{" + '"input": ' + f'"{text}"' + "}"
    data = data.encode("utf-8")
    try:
        ret = requests.post(url, headers=headers, data=data, timeout=5)
        flagged = ret.json()["results"][0]["flagged"]
    except requests.exceptions.RequestException as e:
        flagged = False
    except KeyError as e:
        flagged = False

    return flagged


def pretty_print_semaphore(semaphore):
    if semaphore is None:
        return "None"
    return f"Semaphore(value={semaphore._value}, locked={semaphore.locked()})"
