find "$PWD/src" -type f -name package.xml | while read pkg; do
  dir=$(dirname "$pkg")
  touch "$dir/COLCON_IGNORE"
done
