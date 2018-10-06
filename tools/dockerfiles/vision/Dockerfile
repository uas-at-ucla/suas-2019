FROM python:3.7-slim

# install dependencies
RUN apt-get update
RUN apt-get install -y libglib2.0-0 libsm6 libxrender1 libxext6 libtk8.6 git cython

COPY ./src/vision/build/vision_requirements.txt /vision_requirements.txt
RUN pip install -r /vision_requirements.txt

# build and install darkflow
RUN pip install Cython
RUN git clone https://github.com/thtrieu/darkflow
RUN pip install /darkflow

# copy needed files
COPY ./src/vision /suas/src/vision
COPY ./lib /suas/lib

# expose vision server and ssh
EXPOSE 8099 22

VOLUME ["/suas/src/vision/data_local"]

WORKDIR /suas/src/vision
ENTRYPOINT ["python3", "vision.py"]
