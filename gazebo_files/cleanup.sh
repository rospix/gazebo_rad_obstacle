#!/bin/bash

for f in */*.erb; do
  echo "Removing ${f:0:-4}"
  rm -f ${f:0:-4}
done

echo "Done"
