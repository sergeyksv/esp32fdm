FROM python:3.12-alpine

COPY hub.py /app/hub.py

ENV PYTHONUNBUFFERED=1
EXPOSE 8787

HEALTHCHECK --interval=60s --timeout=5s --retries=3 CMD \
    python -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8787/healthz', timeout=3)"

CMD ["python", "/app/hub.py", "--env", "/config/telegram.env"]
