Ok so currently the bias estimate goes to NaN. Right now it seems to be
diverging if I remove the attitude step. Should this be expected?


# Propagation
Attitude propagation actually looks really good. I tested on fake data
and it was nearly perfect. On actual data it behaves the way we would
expect it to - it just slowly adds to the attitude as we would expect
it to for a small bias estimate. It appears to stay normalized pretty
well.

