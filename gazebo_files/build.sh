#!/bin/bash

./cleanup.sh

for filename in *; do
  if [[ -d $filename ]]; then
    cd $filename
    for f in *; do
      echo $f
      if [[ $f == *".erb"* ]]; then
        echo "Generating ${filename}/${f:0:-4}"
        erb "$f" > ${f:0:-4}
      fi
    done
    cd ..
  fi
done

echo "Done"
