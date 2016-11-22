#!/bin/bash


bash debian/prerm
make uninstall
bash debian/postrm
ldconfig