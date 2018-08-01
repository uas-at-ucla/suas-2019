FROM python:3.6-slim

RUN apt-get update
RUN apt-get install -y libglib2.0-0 libsm6 libxrender1 libxext6 libtk8.6

COPY ./src/vision /suas/src/vision
COPY ./lib /suas/lib
WORKDIR /suas/src/vision

RUN pip install -r build/vision_requirements.txt

EXPOSE 8099 22

VOLUME ["/vision/data_local"]

ENTRYPOINT ["python3", "vision.py"]
