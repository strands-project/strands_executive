#!/bin/bash

cd $1
echo "GOing to " $1
for lib in lib*.a ; do
  target=`echo $lib | cut -d '-' -f 1`
  echo ln -sf $lib $target.a
  ln -sf $lib $target.a
done

for lib in lib*gnu.opt.so ; do
  target=`echo $lib | cut -d '-' -f 1`
  echo ln -sf $lib $target.so
  ln -sf $lib $target.so
done