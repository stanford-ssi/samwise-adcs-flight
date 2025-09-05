# Git Best Practices

Development workflow guidelines for ADCS team members.

## Branch Naming

Use descriptive branch names that match your work scope:

```bash
# Driver development
drivers/magnetometer
drivers/watchdog
drivers/ads7830

# GNC algorithms
gnc/bdot-improvements
gnc/attitude-filter

# Software tasks
software/scheduler-refactor
software/error-handling

# States
states/detumble-tuning
states/nadir-pointing

# Documentation
docs/api-reference
docs/getting-started
```

## Basic Workflow

1. **Create descriptive branch**
   ```bash
   git checkout -b drivers/magnetometer
   ```

2. **Make focused commits**
   ```bash
   # Work on magnetometer driver only
   git add src/drivers/magnetometer.cpp src/drivers/magnetometer.h
   git commit -m "Add RM3100 magnetometer SPI interface"
   ```

3. **Push your branch**
   ```bash
   git push -u origin drivers/magnetometer
   ```

4. **Create pull request** targeting `main`

## Commit Guidelines

- **One feature per branch** - Don't mix magnetometer and IMU changes
- **Atomic commits** - Each commit should build and work
- **Clear messages** - Describe what, not how
- **Test before pushing** - Ensure code compiles and runs

```bash
# Good commits
git commit -m "Fix magnetometer SPI timeout handling"
git commit -m "Add BMI270 gyroscope calibration"
git commit -m "Tune B-dot controller gains"

# Bad commits  
git commit -m "stuff"
git commit -m "fix things"
git commit -m "WIP various changes"
```

## Pull Request Process

1. **Small, focused PRs** - One driver or algorithm at a time
2. **Clear description** - What does this change?
3. **Test results** - Include serial output or test data
4. **Code review** - Wait for approval before merging
5. **Clean merge** - Squash commits if messy

## What NOT to Do

- ❌ Push directly to `main`
- ❌ Mix unrelated changes in one branch
- ❌ Commit broken/non-compiling code
- ❌ Force push to shared branches
- ❌ Leave debugging code or commented sections

## Emergency Fixes

For critical bugs affecting flight hardware:

```bash
git checkout -b hotfix/power-monitor-crash
# Fix the issue
git commit -m "Fix ADM1176 divide-by-zero crash"
# Fast-track review and merge
```

## Branch Management

- Delete branches after merge
- Keep `main` stable and deployable
- Regularly sync with upstream:
  ```bash
  git checkout main
  git pull origin main
  git checkout your-branch
  git rebase main
  ```

**Remember: Clean git history makes debugging easier when things go wrong in space.** 🛰️