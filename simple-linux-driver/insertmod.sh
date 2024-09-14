#/bin/bash
#Author: Roche Christopher

if [ -z $1 ]; then
	echo "Please enter the major number of the device as the first argument"
	exit 1
fi

modulename=simplelindriver
devicename=/dev/simplelinuxdevice1 
owner=whiterose

if lsmod | grep -q "^$modulename"; then
    echo "Module $modulename already exists. So, removing it."
    rmmod $modulename
fi

echo "Inserting module"
insmod src/$modulename.ko

if [ -c $devicename ]; then
	echo "File $devicename exists. So going to delete it"
	rm $devicename
	echo "File $devicename delete"
fi

echo "Creating file $devicename"
mknod $devicename c $1 0
echo "File created"
echo "Changing the permissions of the file"
chown $owner:$owner $devicename
