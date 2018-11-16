simple-power-pmac is a basic IOC to drive 8 axes on a power pmac

To run up:-

cd simple-power-pmac/
grep TODO . -r
edit the files listed by grep
run make

Launch the edm screens:
bin/linux-x86_64/stlab-gui &

Launch the example IOC
bin/linux-x86_64/stlab.sh

Errors relating to BRICK1:COORDINATE_SYS_GROUP are benign and can be ignored.
(this error will be fixed in a later driver release)

