
TAG=2.0.0-hadoop3.2.0-java8




build() {
    NAME=$1
    IMAGE=hadoop-$NAME:$TAG
    FILE=hadoop-$NAME.Dockerfile
    echo '--------------------------' building $IMAGE in $(pwd)
    docker build  -f $FILE -t $IMAGE .
}

#

build base
build namenode
build datanode
# build resourcemanager
# build nodemanager
# build historyserver
# build submit

