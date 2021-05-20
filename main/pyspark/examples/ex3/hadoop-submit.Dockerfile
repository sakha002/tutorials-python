FROM hadoop-base:2.0.0-hadoop3.2.1-java8


COPY hadoop-scripts/WordCount.jar /opt/hadoop/applications/WordCount.jar

ENV JAR_FILEPATH="/opt/hadoop/applications/WordCount.jar"
ENV CLASS_TO_RUN="WordCount"
ENV PARAMS="/input /output"

ADD hadoop-scripts/submit-run.sh /submit-run.sh
RUN chmod a+x /submit-run.sh

CMD ["/submit-run.sh"]
