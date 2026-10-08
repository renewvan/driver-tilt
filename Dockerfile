FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY node_tilt/ node_tilt/
COPY config.default.ini .

ENTRYPOINT ["python", "-m", "node_tilt.main"]
CMD ["--config", "/app/config.ini"]
