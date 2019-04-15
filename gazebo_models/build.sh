#!/bin/bash

for f in */*.erb; do
  echo "Generating ${f:0:-4}"
  erb "$f" > ${f:0:-4}
done
echo "Done"

