# A suite of localization nodes

### Dependency
- GeographicLib
```
sudo apt install geographiclib-tools
sudo apt install libgeographic-dev
```

- for magnetic model installation please check this link
```
https://geographiclib.sourceforge.io/C++/doc/magnetic.html
```
- download the `WMM2020` model then do the following
```
sudo mkdir -p /usr/local/share/GeographicLib
sudo tar xofjC wmm2020.tar.bz2 /usr/local/share/GeographicLib
```
- you may need to rename the file, e.g.,
```
sudo cp wmm2020.wmm WMM2020.wmm
```