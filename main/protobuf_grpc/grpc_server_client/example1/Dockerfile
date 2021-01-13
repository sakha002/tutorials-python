FROM ubuntu


ADD ./install_python.sh .
RUN chmod +x ./install_python.sh
RUN ./install_python.sh


RUN pip3 install grpcio && pip3 install grpcio-tools

ADD ./unary.proto .
RUN python3 -m grpc_tools.protoc -I=. ./unary.proto --python_out=. --grpc_python_out=.

ADD ./client.py .
ADD ./server.py .




