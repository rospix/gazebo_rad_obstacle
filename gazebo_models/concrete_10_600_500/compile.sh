#!/bin/bash

echo "Generating model.sdf"
erb model.sdf.erb > model.sdf
echo "Generating model.config"
erb model.config.erb > model.config
echo "Done"

