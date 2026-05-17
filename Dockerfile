FROM python:3.12-alpine
RUN pip install --no-cache-dir paho-mqtt==2.1.0
WORKDIR /app
COPY bridge.py .
CMD ["python", "-u", "bridge.py"]
