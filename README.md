# sbpl_lattice_planner
*Fork of sbpl_lattice_planner used by Team Hector

# Installing

## SBPL 

Define place where you want to store a sbpl sources:
```
export SBPL_DIR=<sbpl_dir>
```

Download library:
```
mkdir -p "$SBPL_DIR"
git clone https://github.com/sbpl/sbpl.git "$SBPL_DIR"
mkdir "$SBPL_DIR/build"
cd "$SBPL_DIR/build"
cmake .. -G 'Unix Makefiles'
```

### or with install prefix
```
cmake .. -G 'Unix Makefiles' -DCMAKE_INSTALL_PREFIX="../install"
```

### Bulid and install
```
cmake --build . --target install --config Release
```

# Running
## example 

```
roslaunch sbpl_lattice_planner example_node.launch
```

# Debuging 

## SBPL
change DEFINE from sbpl/src/include/sbpl/config.h

to:
```
#define DEBUG 1
```