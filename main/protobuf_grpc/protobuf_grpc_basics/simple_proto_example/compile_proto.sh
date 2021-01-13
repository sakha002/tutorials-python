#!/bin/bash

protoc -I=. --python_out=. ./todolist.proto