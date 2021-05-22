FROM hadoop-base:2.0.0-hadoop3.2.1-java8


HEALTHCHECK CMD curl -f http://localhost:8042/ || exit 1

ADD hadoop-scripts/nodemanager-run.sh /nodemanager-run.sh
RUN chmod a+x /nodemanager-run.sh

EXPOSE 8042

CMD ["/nodemanager-run.sh"]
