#! /bin/bash
LINUX_EXTENSIONS_INSTALL=(
   "GitLab.gitlab-workflow"
)
LINUX_EXTENSIONS_UNINSTALL=(
   "spadin.remote-x11"
)

if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null ; then
   echo "Running on Windows via WSL"
   echo "No need to alter any extension installations"
else
   echo "Running on native Linux"
   echo "Installing VS-Code-Extensions unique to Linux"
   for extension in "${LINUX_EXTENSIONS_INSTALL[@]}"
   do 
      code --install-extension "$extension"
   done 
   for extension in "${LINUX_EXTENSIONS_UNINSTALL[@]}"
   do 
      code --uninstall-extension "$extension"
   done 
fi