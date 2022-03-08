
# import issues 
- another seemingly obvious, seemingly unrelevant question is that how should i organize the files, and how to reference from the structure to the struct utils. the well worn question of how to import in python that i still have issues with.


well besides these notes, there are some great stack responses, that was very comrehensive and I should add the links here for my reference.
I am pretty sure that I have those links and searches, written somewheere in these notebooks, but I put this reminder here for now.
## library imports; what I learned so far:


### reference from the library root

- okay for library, I was referencing (i.e importing) in all files based on the root of the repo, i.e. using the library name (full path). this would be the same way if i wanted to reference (import) to that library installed somewhere else. (also whenever, I wanted to test run, debug, or run pytest, the WD was set at the repo root.)
  
  - making this above assumption, i.e. that people would run & test the codes, from the root of repo as WD seems reasonable, right?. i.e. if someone (like me), were to traverses into the repo to a particular example or script, and gets error, we wouldn't be blamed, right?
    - well, based on what comes next, this practice of running and testing codes from the repo root, does not seem to be so strict,  at least for this kind of importing, as long as the repo root is somehow added to the PATH. I guess the CWD, it only could become problematic, if there is some sort of file-read/write in the code that is is doing relative path. (I am doing this a lot, not good huh?!)
    - it is good i guess, to have the code in a way that does not matter from where you call it.
  
- **Question** so for a long time I was actually running the scripts from their own directories, (i guess I am still doing that, for the applications, like these server, client scripts, so is there a different situation there? is it okay? and why?)
    - again, based on above, I think the reason that those work, is that (i) in those particular application I have assumed all the relative addresses based on the script CWD, and also the all the imports are in the same directory or below it. (ahh this is exactly the kind of practice that I was thinking about, it works but it is a bad idea)
  
- with this assumption on importing, you would need to have the repo root path, to be added to the python path, i.e. if you are working on exlib, the python should recognize "import exlib" when running tests or examples.
  
- so for this latter, we used to have a few different practices, some worse than the others!

- so the final note here for (full path imports) is I guess for the imports as long as the repo root is added to the path, and that addition is independent of the CWD, then it does not matter how or from where you run the code. ( this would be also the case for loading/reading a file, etc., it should not (preferably) depend on the CWD)


### how to introduce Repo root path, for testing and debug
-  I guess one practice that we had in using the vscode debug mode, was to introduce the library path or the module path, as env variable in debug settings file.
    ```
    "cwd": "/home/wd/projects/lib1",
    "env": {"PYTHONPATH": "/home/wd/projects/lib1"},

    ```

- the other way which would solve the issue here (for libraries) was to pip install -e  the library from the repo address, while developing on it. I am still doing this, since this would make it accessible for other repos (applications or libraries) which depend on it.
  
  - but I guess for the purpose of testing and debugging only, this would not be generally the good way, also I guess it won't work for applications.


- In general, I think you don't want to be relying on installing a library like that when you are developing. So then with that given, how you would go? I guess then these "correct WD" and also the "context.py" could become relevant.
  

- Also it is the case for the application type modules, when essentially there will not be a pip install at all, but instead there will be some entrypoint script, that is calling the app. So do we put that script in the root of repo? 

**will dive into this more later**

- This context.py file that we add helps to be able to run the scripts from any CWD. where the library/application path is added at runtime. it would be like "from context import optopy". then adding lines such as:

```
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import library

```
in this one, the library root is one level above the test directory/files.

another one:

```
#context.py

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../mip_client/"))
)


from mip_client_utils import (
    MipModel,
    MipVariableArray,
)

```

and then in the test file

```
from context import (
    MipModel
)

```
but I guess this would be better  if I had  changed it into:


```
#context.py

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../mip_client/"))
)


import  mip_client_utils 
```
then in the second file I would have:


```
from context import mip_client_utils

from mip_client_utils import(
    MipModel,
    MipVariableArray,
)

```



- **Question** If we have the CWD set at the repo root, does it mean that we will not need to have those context.py setup and all?
    - I think yes, I should check it.




## import rules when you have your repo structured with "src"

so somewhere I saw an article suggesting to use this structure instead, with a few good arguments.(TODO include the ref)
```
-root
    |
    |--src
    |    |
    |    |-libname
    |         |     
    |         |-module1
    |         |
    |         |-module2
    |
    |-tests
    |    |
    |    |-test_module1
    |    |-test_module2
    |
    setup.py
```

**will Look into this later**

## file imports; my mistakes and lessons

besides the importing that becomes troublesome, the other potential is when loading some files in the codes, e.g. in the examples, or test scripts.
  
- a useful command in this regard: pathlib.Path(__file__).parent

- so when loading a file in the code, I used to write the relative path for the file path to load that file. and with that if I gave the address of file as "./path/to/file.csv" then I used to have the assumption of path of current script as the CWD. The code  would run with no problem (I think), since I had used a pip install -e, or used the contex.py so the imports were working okay.
 
- but I guess if someone does not wish to use something like that sort of Path(__file__), and just give some relative path, it is best to assume the script is run from the repo root (well now at least for  libraries)
  - well If I were to give an address like "lib_name/path/to/file"

- I don't know that if in fact for some time this assumption of running the codes with wd set as the file directory was only my practice or others were doing that as well. 

  
  
## import resolutions with pytest


- These notes above around calling the test scripts and functions seems to be relevant when manually running the tests, or in the debug utils of vscode, etc., but for using the pytest, it has its own mechanims to figure out the imports etc. 
  
- for pytest  need to add an empty file "conftest.py" to the main directory of the library (i.e if library name is mylib, it should be inside mylib dir)

- looks like we put it where we basically want it to be main directory of the application/library

```
    repo_root
        |
        |- library_name
        |        |
        |        |- __init__.py
        |        |
        |        |- module1
        |        |
        |        |- module2
        |        |
        |        |- docs(?)
        |        |
        |        |- examples(?)  
        |        |
        |        |-sample_data  
        |        |
        |        |- conftest.py
        |
        |-tests
        
```

**Question** should docs and examples appear in the first level, i.e. root or in the second one?

## importing story in the applications


- now in this current case of interest, since it is an application, we don't have a particular library name, usually our practice was to have a  "main"  (or src) directory and put all the application modules under it. 

- sometimes, we used to have several totally stand-alone modules, that were developed seperately, and maybe even different language, but we used to call them inside the container with some script, or DAG, etc.

- so for these kind of repos, where you are working on some sub-application, or module, how we would go about importing and loading files?

- should we still go from root like 
    ```
    from src.application1 import blah

    from main.application1.module1 import blahblah
    ```
    nah,  that does not seem right!

- what we used to do was to reference from one level below, i.e.
    
    ```
    from application1 import blahbal
    ```

- so then the context.py stories, and file relative paths, will be all the same here, only based on src/main as CWD.

- one other consideration here is that if we have several stand alone applications/modules, then inside src/main we would want each to be containing all of their files separate, including test, so this would be a bit different than a single application. that is we have:

```
    - src
        |
        |
        |-module1
        |
        |-module2
        |
        |-tests
        |
        |-sample_data
        |
        |-documentation(?)
        |
        |-conftest.py(?)
```

another alternative for this one above (which I am not sure if it is preffered over the first one would be:

```
repo_root
    |
    |- src
        |
        |-Application1
        |       |
        |       |-module1
        |       |
        |       |-module2
        |       |
        |       |-data
        |       |
        |       |-conftest.py(?)
        |    
        |-tests    
        |
        |-docs(?)

```      

This second one would be more similar to the organization of a library repo.
not sure at all if the "docs" shouldn't be at  the repo root or not?

for multi app we will have it like:

```
repo_root
    |
    |- src
        |
        |
        |-application1
        |       |
        |       |-module1
        |       |
        |       |-tests
        |       |
        |       |-sample_data
        |       |
        |       |-conftest.py(?)
        |
        |-application2

```

okay so quite a few question marks here are left.


oh well in any and all of these cases, I guess the rule for referencing from the src/main would still hold for full path imports. (and relative file paths)

now i guess it's good to visit the topic of relative importing as well.

## .. Relative imports

when putting a "from .some_modules" this little dot would create a whole lot of troubles. and it was confusing for me for a long while. usually I have been trying to avoid it. but let me see if I can find that link which explained it well, [here](https://stackoverflow.com/questions/14132789/relative-imports-for-the-billionth-time)

so it seems that the dot is interperated in two different ways depending on if it in the main script/test code or if it is within a module.
>There are two ways to load a Python file: as the top-level script, or as a module.
>It is loaded as a module when an import statement is encountered inside some other file.


> Naming:
When a file is loaded, it is given a name (which is stored in its __name__ attribute). If it was loaded as the top-level script, its name is __main__. If it was loaded as a module, its name is the filename, preceded by the names of any packages/subpackages of which it is a part, separated by dots.

So for instance in your example:
```
package/
    __init__.py
    subpackage1/
        __init__.py
        moduleX.py
    moduleA.py
```
if you imported moduleX (note: imported, not directly executed), its name would be package.subpackage1.moduleX. If you imported moduleA, its name would be package.moduleA. However, if you directly run moduleX from the command line, its name will instead be __main__, and if you directly run moduleA from the command line, its name will be __main__. When a module is run as the top-level script, it loses its normal name and its name is instead __main__.

Accessing a module NOT through its containing package

```
There is an additional wrinkle: the module's name depends on whether it was imported "directly" from the directory it is in, or imported via a package. This only makes a difference if you run Python in a directory, and try to import a file in that same directory (or a subdirectory of it). For instance, if you start the Python interpreter in the directory package/subpackage1 and then do import moduleX, the name of moduleX will just be moduleX, and not package.subpackage1.moduleX. This is because Python adds the current directory to its search path when the interpreter is entered interactively; if it finds the to-be-imported module in the current directory, it will not know that that directory is part of a package, and the package information will not become part of the module's name.
```
Relative imports...

Relative imports use the module's name to determine where it is in a package. When you use a relative import like from .. import foo, the dots indicate to step up some number of levels in the package hierarchy. For instance, if your current module's name is package.subpackage1.moduleX, then ..moduleA would mean package.moduleA. For a from .. import to work, the module's name must have at least as many dots as there are in the import statement.

However, if your module's name is __main__, it is not considered to be in a package. Its name has no dots, and therefore you cannot use from .. import statements inside it. If you try to do so, you will get the "relative-import in non-package" error.

**Important:**
**Scripts can't import relative**

```
Note that this error will also happen if you run Python from the same directory where a module is, and then try to import that module, because, as described above, Python will find the module in the current directory "too early" without realizing it is part of a package.
```

If you really do want to run moduleX directly, but you still want it to be considered part of a package, you can do python -m package.subpackage1.moduleX. The -m tells Python to load it as a module, not as the top-level script.

```
Or perhaps you don't actually want to run moduleX, you just want to run some other script, say myfile.py, that uses functions inside moduleX. If that is the case, put myfile.py somewhere else – not inside the package directory – and run it. If inside myfile.py you do things like from package.moduleA import spam, it will work fine.
```

(I guess this will be the most confusing case, the moduleX is developed and has its own relative imports, which would work fine, but if we call a script in that same directory, all of those relative imports would run into error)

so consuing enough still! and I guess I have read this how many times (and I am sure i will get back to it more and more)

so again as some elder said in this same link, the solution is simple, don't do relative imports! (if possible)



## Update on repo structure


so on the question of where to put tests, I went over two of my favourite Porjects (or-tools and tensorflow).
I saw some interesting patterns.
In the or-tools they had put the tests as part of the examples, which makes sense to me since at least in my case a lot of these test has some elements in common with examples.
In the TF repo, they did not seem to have a separate directory in the top levels and instead it seemed it was dispeared over the different pieaces.
i.e for example Keras had its own test directory and some of the tests where just alongside the actual codes.

So if we have a large library/package, I guess putting the test


In ortools  I saw docs and examples at the top level.
in TensorFlow, I found examples in the 2nd level i.e. under the tensorflow, and the docs were an entire seperate repo.

so overall seems, whatever fits, it fits!


## VERY IMPORTANT UPDATE:


The key here is that, by default, Python includes the folder of the script in its search path, BUT NOT THE CWD. 




# Update August 2021

So this is still the question on correct way of seeting up references (imports and file read/write) to work properly on testing and third environments. The focus on Applications.

## the question is from src or from root

- pytest figures out how to interpret refs based the conftest.py, so it is not an issue there
- imports can be figured out to be indifferent by using the context.py.
-  for debuging, VSCODE assumes the directory of current window  as wd.
-  the repo root seems a safer option.
-   still I like better to take "src" as the point of reference, then it means that I won't have to write imports like "from src.blah import blahblah"
-   what is the problem with this?
    -    it would be mostly for reading/writing files that have "." or "cwd" used in them somehow.
    -   The user, would need to run the scripts from the src (i.e. cwd as src).
    -   if one go to the directory of the script and  run it or run it from the repo root, it won't work.
    -   this could become a scneario only for the testing, i.e. when a third party tries to run/test the script.
    -   It would be good if could make file read paths to be based on the "file" not cwd.
    -  talking to coleagues and veterans seems like having the src as the reference for imports is the actual best practice for other languages as well.
  
