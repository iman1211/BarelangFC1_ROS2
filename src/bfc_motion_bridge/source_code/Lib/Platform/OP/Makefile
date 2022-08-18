CWD= $(shell pwd)

all:: body kinematics

body::
	cd Body && make && cd $(CWD)
kinematics::
	cd Kinematics && make && cd $(CWD)

clean:
	cd Body && make clean && cd $(CWD)
	cd Kinematics && make clean && cd $(CWD)


