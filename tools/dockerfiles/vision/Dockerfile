FROM python:3.6-slim

# Install apt-fast for faster APT downloads. ###################################
WORKDIR /tmp
RUN git clone https://github.com/ilikenwf/apt-fast.git
RUN cp apt-fast/apt-fast /bin                                               && \
    chmod +x /bin/apt-fast                                                  && \
    cp apt-fast/apt-fast.conf /etc                                          && \
    echo "VERBOSE_OUTPUT=n" > /etc/apt-fast.conf

# install dependencies
RUN apt-get update     -qq                                                  && \
    apt-get install -y -qq                                                     \
    libglib2.0-0                                                               \
    libsm6                                                                     \
    libxrender1                                                                \
    libxext6                                                                   \
    libtk8.6                                                                   \
    git                                                                        \
    cython

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
# if you leave out the -u argument, sometimes print statements are deferred
# until after the docker container is killed
ENTRYPOINT ["python3", "-u", "vision.py"]
