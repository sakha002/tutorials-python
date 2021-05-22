FROM hadoop-base:2.0.0-hadoop3.2.0-java8-m


HEALTHCHECK CMD curl -f http://localhost:9864/ || exit 1

ENV HDFS_CONF_dfs_datanode_data_dir=file:///hadoop/dfs/data
RUN mkdir -p /hadoop/dfs/data
VOLUME /hadoop/dfs/data

ADD hadoop-scripts/datanode-run.sh /datanode-run.sh
RUN chmod a+x /datanode-run.sh

EXPOSE 9864

CMD ["/datanode-run.sh"]
