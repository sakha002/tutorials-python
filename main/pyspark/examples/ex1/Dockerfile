FROM ubuntu


ADD ./install_python.sh .
RUN chmod +x ./install_python.sh
RUN ./install_python.sh

RUN  export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -y  && \
    apt-get install -y default-jre && \
    apt-get install -y scala 


RUN pip3 install py4j


RUN wget https://downloads.apache.org/spark/spark-3.1.1/spark-3.1.1-bin-hadoop2.7.tgz

RUN tar -zxvf spark-3.1.1-bin-hadoop2.7.tgz



ENV SPARK_HOME='/spark-3.1.1-bin-hadoop2.7' 
ENV   PATH=$SPARK_HOME:$PATH 
ENV  PYTHONPATH=$SPARK_HOME/python:$PYTHONPATH





