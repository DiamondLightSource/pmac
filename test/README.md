# How to Run pmac module system tests

## Start up the example IOC

```bash
./iocs/lab/bin/linux-x86_64/stlab-gui &
./iocs/lab/bin/linux-x86_64/stlab.sh
```
The IOC needs to connect to a test pmac turbo clipper which is expected to
be at 172.23.240.97.

Modify `etc/makeIocs/lab.xml` to make any changes like IP address and 
rebuild with
```bash
cd pmac  # root of the project
make IOCS=lab
```

The clipper needs to have the default configuration applied to it.
See [here](../etc/bootstrap/README.md) for details.

## setup the test virtualenv and execute the tests

```bash
cd pmac\test
pipenv install --dev --python dls-python3

# launch the pmac system tests
pipenv run python -m pytest test_pmac/
```

Note there are also Malcolm tests which require changes to the Pipfile
to bring in more dependencies. See comments in Pipfile.

