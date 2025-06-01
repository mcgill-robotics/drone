# Scripts and models needed to generate the dataset
## How to run ?
1. You need Blender (only tested with 4.3).
2. Add Blender to your path variable.
3. Run `blender -b -P generate.py`
4. Sit back and enjoy
## How to modify the code ?
### Modify params
The major parameters, such as number of instances and such are global variables in all caps at the top of the generate file
### Other stuff
The dynamic module import is needed because of blender and the `helpers` module not being in blender's path (I couldn't be bothered to look into it)
