FROM ubuntu

ADD ./install_python.sh .
RUN ./install_python.sh

ADD ./install_protobuf_compiler.sh .
RUN ./install_protobuf_compiler.sh

ADD ./addressbook.proto . 

ADD ./compile_proto.sh . 
RUN ./compile_proto.sh

ADD ./create_addressbook.py .
ADD ./read_addressbook.py . 




