inline float lookup(
    float x,
    const float *xt,
    const float *yt,
    int n)
{
    if (x <= xt[0])
        return yt[0];

    if (x >= xt[n - 1])
        return yt[n - 1];

    for (int i = 0; i < n - 1; i++)
    {
        if (x <= xt[i + 1])
        {
            float f =
                (x - xt[i]) /
                (xt[i + 1] - xt[i]);

            return yt[i] +
                   f * (yt[i + 1] - yt[i]);
        }
    }

    return yt[n - 1];
}
