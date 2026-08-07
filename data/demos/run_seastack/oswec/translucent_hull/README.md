# OSWEC translucent flap

Still-water free flap with `bodies[].visualization.opacity: 0.35` on the flap
mesh so the base and hinge stay visible in the VSG GUI.

## Run

```
build\bin\Release\run_seastack.exe data\demos\run_seastack\oswec\translucent_hull
```

Use `--nogui` for a headless smoke run. Opacity applies to **mesh** visuals
(`model_file`) only; Chrono primitive `shapes:` are unchanged.
