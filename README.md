# iot
IOT module for AMD-R

# Parameters
## Config File
The http client for the iot system requires a YAML configuration file to work.
The configuration can be set by creaing a `cfg/server.yml` file or generated by the AMD-R`welcome` package
An example `server.yml` file is shown below:

``` yaml
host: 'http://localhost'
port: 5000
# A file name set by private_key and public_key must exist in the same directory containing it's respective key
private_key: 'private.pem'
public_key: 'public.pem'
# A password for the private key is not needed, leave it empty if there is none
password: 'password'
```

## Verbose
The verbose parameter can be set to `1` or `0` to determine if change the verbosity of the node
(Logging what data it is sending). A value of `1` means it will verbose the message send and `0` will not.

# Usage
## IOT
To run all the nodes required for the IOT part of the AMD-R run:
``` sh
roslaunch iot iot.launch
```

## HTTP Client
To only run the node for http client run:

``` sh
roslaunch iot http.launch
```
