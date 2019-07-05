FROM python:3.7-slim

ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

WORKDIR /uas

RUN pip install pipenv
RUN apt-get update
RUN apt-get -y install gcc libglib2.0 libsm6 libxrender1 libxext6
COPY Pipfile Pipfile.lock /uas/
RUN pipenv install --deploy --ignore-pipfile

COPY . /uas
