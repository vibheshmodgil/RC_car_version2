# The PC's half of the truck: speech (tts_server.py) and object detection
# (detect_server.py) in one image. Built from the repo root so it can take
# test/tts.py — the speech server reuses the Pi's own text splitting and
# voice list. Run it with docker-compose.yml in this folder.
#
# Ollama is deliberately not in here: it already runs natively on Windows
# with the GPU, and a GTX 1650's 4 GB is all needed by the language model.
FROM python:3.12-slim

# git: ultralytics installs CLIP from GitHub for the open-vocabulary model.
# libgl1/libglib2.0-0: OpenCV, which ultralytics imports.
RUN apt-get update \
 && apt-get install -y --no-install-recommends git libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# CPU-only torch: ~200 MB instead of ~2.5 GB of CUDA the GPU could not spare.
RUN pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu
RUN pip install --no-cache-dir "piper-tts>=1.3" numpy pillow ultralytics \
    "git+https://github.com/ultralytics/CLIP.git"

WORKDIR /app
COPY test/tts.py tools/tts_server.py tools/detect_server.py ./
ENV PYTHONUNBUFFERED=1
