FROM nginx


COPY hadoop-scripts/default.conf /etc/nginx/conf.d/default.conf
COPY hadoop-scripts/materialize.min.css /data/bde-css/materialize.min.css
COPY hadoop-scripts/bde-hadoop.css /data/bde-css/bde-hadoop.css
