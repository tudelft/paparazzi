
How to make a new release
=========================

1. Update the vesion number in CMakeLists.txt
2. Update `debian/changelog`: run `dch` from the root directory.
3. Makes sure the author name and email matches one of your gpg keys (get the list with `gpg --list-secret-keys`)
4. `debuild -S`
5. `dput ppa:paparazzi-uav/ppa <package>.changes`
6. Update minor version number and distribution name to build for other distribution, and restart from step 4. Do not commit those changes.
