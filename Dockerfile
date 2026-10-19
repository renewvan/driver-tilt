FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY node_tilt/ node_tilt/
COPY manifest.json node_tilt/manifest.json
COPY config.default.ini .

# The git release tag, passed by release.yml; the only version source.
# Unset (a local build) the node reports `dev`.
ARG NODE_VERSION
ENV NODE_VERSION=${NODE_VERSION}

ENTRYPOINT ["python", "-m", "node_tilt.main"]
CMD ["--config", "/app/config.ini"]
