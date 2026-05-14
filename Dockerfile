FROM python:3.12-slim

WORKDIR /app

# System dependencies for matplotlib
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc libffi-dev && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY analyze.py ardupilot_error_codes.py ./
COPY web/ web/

# Create upload dirs
RUN mkdir -p web/uploads

ENV PORT=8040
EXPOSE 8040

CMD uvicorn web.app:app --host 0.0.0.0 --port ${PORT}
