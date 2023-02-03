# Farscope Industry Project

This repository keeps software and hardware stuff related to the Amazon Picking Challenge, as part of the work of the 2023 Farscope Industry Project. The project is developed by the 2022 cohort working in two sub-teams as follows:

### Sensing Team
This teams focuses on the vision system of the project, developing machine vision algorithms to process information coming from a camera. The members are:

- Emmanuel Akirintoyo
- Ben Allen
- Reece White
- Ajmal Roshan

### Hardware and Control Team
This team focuses on the gripper design and implementation, as well as on the control and motion planning of the robotic arm. The members are:

- Isabelle Ormerod
- Audric Tamby
- Ugnius Bajarunas
- Sergio Sierra


## How to use this repository
To keep a clean workflow multiple code versions are going to maintained using different branches. The main branch should only be used for the "production" or latest working version of the code. Please always follow these steps:

1. Pull/update the latest version in the repository.
2. Create new branch for new versions or work in progress.
3. Push to the new branch and save the work.
4. Ensure code is tested and ready to be merged.
5. Merge/pull request custom branch to the main one.

## How to install Git
You will need git on your system to be able to interact with these repository.

- For windows users:
Download the last version from [https://git-scm.com/download/win](https://git-scm.com/download/win) and follow the installation instructions. If everything works fine you will be able to open a console and execute git commands.

- For mac/linux users:
It is highly probable that git is already installed, otherwise use your preferred package manager to install it:
    - mac: `brew install git`
    - linux: `sudo apt install git`

## How to clone a repository
1. On GitHub.com, navigate to the repository main page.
2. Above the list of files, click  Code.
3. Copy the URL for the repository. Make sure you are under HTTPS tab.
4. Open a terminal/console.
5. Go to the directory where you want to keep the repository.
6. Type: `git clone https://github.com/YOUR-USERNAME/YOUR-REPOSITORY`.
   In this case, `https://github.com/sergiosierram/farscope_industry_project.git`

## How to update a repository
Git pull updates your current local working branch, and all of the remote tracking branches. It's a good idea to run git pull regularly on the branches you are working on locally. Basic usage:

1. Open a terminal.
2. Go inside the repository folder.
3. `git pull` 

## How to checkout to a new branch
The git checkout command lets you navigate between the branches created by git branch. Checking out a branch updates the files in the working directory to match the version stored in that branch, and it tells Git to record all new commits on that branch. 

### Check existing branches
To find out what branches are available and what the current branch name is, execute:

``` 
$ git branch 
main 
another_branch 
feature_inprogress_branch 
```  

### Create new branch
The following command lets you create a new branch:

``` 
git branch new_branch
``` 

### Checkout to a new branch
The git checkout command switches between branches:

``` 
git checkout other_branch
``` 

The command also accepts a -b argument that acts as a convenience method which will create the new branch and immediately switch to it. 

``` 
git checkout -b new_branch
```

## Comitting and pushing work
Every time your code has new working features you can save your working by commiting the changes. The commit command is frequently used with add command which receives the files that were modified.

1. Add files that were modified: 
    - `git add .` The . option asks git to add the current directory.
    - `git add -a` The -a option asks git to add everything.
2. Commit the changes:
    - `git commit -m "some detailed message"` The -m options add a message to your commit. It is recommended to be concise and descriptive here.
3. Pushing the changes:
    - `git push origin name_of_your_branch` This saves your work in the remote repository.

## Merging
Once your code is tested and ready to be integrated into the latest working version, you should merge it to the main branch. This can be done by two different ways:

1. Git merge: This command merges the content from one branch to another one.
    1. Checkout to the main branch or the branch you want to update.
    2. Merge with: `git merge branch_with_new_features`
    3. If everything goes perfectly, there won't be any issues. If something clashes, it should be reviewed with any merging tool. You can use visual studio.
2. Directly on Github.com go to the branch with the new feature and create pull request.