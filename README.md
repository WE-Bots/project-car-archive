# Project-CAR
This is the private repository for the code for Project CAR.

This repository uses Git LFS for tracking large files. Please install
 Git LFS from [here](https://git-lfs.github.com)
 before proceeding.

This repository's old history was moved to
 [the archive](https://github.com/WE-Bots/Project-CAR-archive).
 Please see the section ***Retreiving Old History***
 for details on how view the old competition history.


## Prerequisite Software
* Git
* Git LFS (Available from [here](https://git-lfs.github.com))


## Cloning
Due to this repository using Git LFS to track large files, a new command
 is used to speed up the initial clone. Use `git lfs clone *repository link*`
 to clone this repository ***significantly*** faster.


## Contributing
The Git repository is structured into two main branches and feature branches:
* The **master** branch contains stable, _(roughly)_ tested code that can be
 loaded onto the CAR for the competition with minimal issues. Code in here is
 expected to be _mostly functional_.
* The **develop** branch contains completed work that has been tested to
 _mostly_ work, but has not been checked to make sure it plays well with the
 rest of the car's systems. 

Critical branches **master** and **develop** are locked to Project CAR team
 leads only. Work on your assigned tasks in a separate branch off of **develop**,
 and submit a _Pull Request_ to have your work merged into **develop**.

Please refrain from adding large files to your commits without getting permission
 from your team lead first (to avoid using up all of our Git LFS storage space).


## Retrieving Old History
To retrieve the previous repository's history, use the following commands:
```
# Add the archive repository to your local history and then get its contents
git remote add history https://github.com/WE-Bots/Project-CAR-archive.git
git fetch history

# Stitch the history back together
git replace origin/history_merge_point history/master
```
To undo this stitching together and get back to work:
```
git replace -d origin/history_merge_point history/master
git remote remove history
```

Source: [https://git-scm.com/blog/2010/03/17/replace.html](https://git-scm.com/blog/2010/03/17/replace.html)
