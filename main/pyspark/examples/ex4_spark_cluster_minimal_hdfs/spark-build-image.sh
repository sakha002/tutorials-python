#!/bin/bash

set -e

TAG=3.1.1-hadoop3.2-m

build() {
    NAME=$1
    IMAGE=spark-$NAME:$TAG
    FILE=spark-$NAME.Dockerfile
    # cd $([ -z "$2" ] && echo "./$NAME" || echo "$2")
    echo '--------------------------' building $IMAGE in $(pwd)
    docker build  -f $FILE -t $IMAGE .
}

build base
build master
build worker
build jupyterlab



