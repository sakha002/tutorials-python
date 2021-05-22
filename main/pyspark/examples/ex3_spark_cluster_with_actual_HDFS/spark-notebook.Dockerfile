FROM hadoop-base:2.0.0-hadoop3.2.1-java8

ENV APACHE_SPARK_VERSION 2.1.0
ENV APACHE_HADOOP_VERSION 3.1.1

RUN set -x \
    && curl -fSL "https://dl.dropboxusercontent.com/u/4882345/spark-notebook/spark-notebook-0.7.0-scala-2.11.8-spark-2.1.0-hadoop-2.8.0-with-hive.tar.gz" -o /tmp/spark-notebook.tgz \
    && tar -xzvf /tmp/spark-notebook.tgz -C /opt/ \
    && mv /opt/spark-notebook-* /opt/spark-notebook \
    && rm /tmp/spark-notebook.tgz

COPY spark-scripts/notebook-run.sh /rnotebook-un.sh
RUN chmod a+x /notebook-run.sh

COPY spark-scripts/application.conf /opt/spark-notebook/conf/
COPY spark-scripts/clusters /opt/spark-notebook/conf/
COPY spark-scripts/profiles /opt/spark-notebook/conf/
COPY spark-scripts/jars /jars

RUN mkdir -p /data/resources

ENV NOTEBOOKS_DIR "/opt/spark-notebook/notebooks"
ENV RESOURCES_DIR "/data/resources"
ENV SPARK_MASTER "spark://spark-master:7077"
ENV SPARK_EXECUTOR_MEMORY "4G"
ENV EXTRA_CLASSPATH "/jars/*"

WORKDIR /opt/spark-notebook/

CMD ["/notebook-run.sh"]
