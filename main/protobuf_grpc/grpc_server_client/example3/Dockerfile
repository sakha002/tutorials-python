FROM ubuntu


ADD ./install_python.sh .
RUN chmod +x ./install_python.sh
RUN ./install_python.sh


RUN pip3 install grpcio && pip3 install grpcio-tools

ADD ./route_guide.proto .
RUN python3 -m grpc_tools.protoc -I=. ./route_guide.proto --python_out=. --grpc_python_out=.

ADD ./client.py .
ADD ./server.py .
ADD ./route_guide_db.json .
ADD ./route_guide_resources.py .

ADD ./python_requirements.txt .
# RUN pip3 install -r ./python_requirements.txt

