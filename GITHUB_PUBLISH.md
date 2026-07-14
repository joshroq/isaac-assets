# Publishing This Repo To GitHub

The top-level repository has been initialized locally on branch `main`.

This workstation does not currently have `gh` installed, so the remote GitHub
repository cannot be created automatically from here. Use either path below.

## With GitHub CLI

```bash
sudo apt-get install gh
gh auth login
cd /home/joshroq/projects/isaac-assets
gh repo create isaac-assets --private --source=. --remote=origin --push
```

Use `--public` instead of `--private` if you want the repo public.

## With A Repo Created In The Browser

Create an empty GitHub repo named `isaac-assets`, then run:

```bash
cd /home/joshroq/projects/isaac-assets
git remote add origin git@github.com:<your-user-or-org>/isaac-assets.git
git push -u origin main
```

## Note About `ugv_ws`

`ugv_ws` already contains its own git repository pointing at
`https://github.com/waveshareteam/ugv_ws.git`. The parent repo currently sees it
as a nested repository. If you want this parent repo to contain `ugv_ws` as plain
files instead of a nested repo/submodule, remove or archive `ugv_ws/.git` first,
then `git add ugv_ws`.

