#!/bin/bash

_PKG=pmc464-dkms
_MODULE=pmc464
_VERSION=1.00.00

# Check the version pattern "X.YY.ZZ"
echo Check the version pattern -------------------------------------------------
if [[ ${_VERSION} =~ [0-9]\.[0-9][0-9]\.[0-9][0-9] ]] ; then
echo ${_MODULE}-${_VERSION}
else
   echo The version does not match with the expected pattern "X.YY.ZZ" ------------
   exit 0
fi

# Generate the dkms.conf file
echo Generate the dkms.conf file -----------------------------------------------
echo 'PACKAGE_NAME='${_MODULE}               > dkms.conf
echo 'PACKAGE_VERSION='${_VERSION}           >> dkms.conf
echo 'CLEAN="make clean"'                    >> dkms.conf
echo 'MAKE="make all KVERSION=$kernelver"'   >> dkms.conf
echo 'DEST_MODULE_LOCATION="/updates"'       >> dkms.conf
echo 'AUTOINSTALL="yes"'                     >> dkms.conf
echo 'REMAKE_INITRD="no"'                    >> dkms.conf
echo 'POST_INSTALL="pmc464_post_install"'    >> dkms.conf
echo 'POST_REMOVE="pmc464_post_remove"'      >> dkms.conf
   
# Remove a previously installed package if exist
if dpkg-query -Wf'${db:Status-abbrev}' "${_PKG}" 2>/dev/null | grep -q '^i'; then
   echo Remove a previously installed package -------------------------------------
   sudo apt-get remove -y ${_PKG}
fi
   
# Copy the files in /usr/src
echo Copy the files in /usr/src ------------------------------------------------
sudo mkdir -p /usr/src/${_MODULE}-${_VERSION}
sudo cp -r ./* /usr/src/${_MODULE}-${_VERSION}/

# Patch template files
sudo mkdir /usr/src/${_MODULE}-${_VERSION}/${_MODULE}-dkms-mkdeb
sudo cp -r /etc/dkms/template-dkms-mkdeb/* /usr/src/${_MODULE}-${_VERSION}/${_MODULE}-dkms-mkdeb
# Comment a line in the Makefile where is fix executable permissions from the entire source tree
sudo sed -i '/chmod 644 -R "$(SRC)[/]$(NAME)-$(VERSION)"/s/^/#/' /usr/src/${_MODULE}-${_VERSION}/${_MODULE}-dkms-mkdeb/Makefile

# Remove unwanted files
echo Remove unwanted files -----------------------------------------------------
rm -f /usr/src/${_MODULE}-${_VERSION}/.git

# Register, build and install the module with dkms
echo Register, build and install the module with dkms --------------------------
sudo dkms add -m ${_MODULE} -v ${_VERSION}
sudo dkms build -m ${_MODULE} -v ${_VERSION}
sudo dkms install -m ${_MODULE} -v ${_VERSION}

# Create the debian package
echo Create the debian package -------------------------------------------------
sudo dkms mkdsc -m ${_MODULE} -v ${_VERSION} --source-only
sudo dkms mkdeb -m ${_MODULE} -v ${_VERSION} --source-only

# Save the generated debian package
echo Save the generated debian package -----------------------------------------
cp /var/lib/dkms/${_MODULE}/${_VERSION}/deb/${_PKG}_${_VERSION}_all.deb .

# Uninstall and unregister the module with dkms
echo Uninstall and unregister the module with dkms -----------------------------
sudo dkms uninstall -m ${_MODULE} -v ${_VERSION}
sudo dkms remove -m ${_MODULE} -v ${_VERSION} --all

# Remove temporary files
echo Remove temporary files ----------------------------------------------------
sudo rm -rf /var/lib/dkms/${_MODULE}
sudo rm -rf /usr/src/${_MODULE}-${_VERSION}

# Install the generated package
echo Install the generated package ---------------------------------------------
sudo dpkg -i ./${_PKG}_${_VERSION}_all.deb
   
# Remove the generated package
echo Remove the generated package ----------------------------------------------
rm ./${_PKG}_${_VERSION}_all.deb
