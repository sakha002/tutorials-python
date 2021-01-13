#!/bin/bash

protoc -I=. --python_out=. ./addressbook.proto