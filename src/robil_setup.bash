function robil_setup_here {
	. devel/setup.bash
}

function robil_eclipse_project {
	PROJECT_NAME=$(basename $(pwd))
	cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8
	mv .project .project~
	cat .project~ | sed 's|^\t<name>.*</name>|\t<name>'"$PROJECT_NAME"'</name>|g' > .project
	rm .project~
	if test -e .gitignore ; then echo; else
		echo "/src/$PROJECT_NAME" > .gitignore 
	fi
	robil_setup	
}

function robil_create_project {
	PROJECT_NAME=$1
	catkin_create_pkg $@
	cd $1
	robil_eclipse_project
}

function robil_fix {
	cmake .
}

function robil_fix_all {
	find . -name "CMakeLists.txt" -type f |
		while read file; do
			pushd .
			cd $( dirname $file )
			echo "==> Fixing project at $(pwd)"
			robil_fix
			popd
		done
}

function robil_clean_builds { 
	rm -rf devel/ build/ install/
}

function robil_regenerate_framework {
	../robil2conf/regenerate.py
}

function robil_regenerate_framework_all {
	find . -name "CMakeLists.txt" -type f |
		while read file; do
			pushd .
			cd $( dirname $file )
			robil_regenerate_framework
			popd
		done
}


