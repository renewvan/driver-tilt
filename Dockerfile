FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY driver_tilt/ driver_tilt/
COPY manifest.json driver_tilt/manifest.json
COPY config.default.ini .

# The git release tag, passed by release.yml; the only version source.
# Unset (a local build) the device reports `dev`.
ARG DEVICE_VERSION
ENV DEVICE_VERSION=${DEVICE_VERSION}

ENTRYPOINT ["python", "-m", "driver_tilt.main"]
CMD ["--config", "/app/config.ini"]
