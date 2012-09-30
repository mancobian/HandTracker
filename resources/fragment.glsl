#version 110

// we will use two 2D textures
uniform sampler2D tex0;
uniform sampler2D tex1;

vec4 lerpColor(vec4 colorA, vec4 colorB, vec4 colorC, float mu)
{
  if (mu <= 0.5)
    return colorA * mu * 2.0 + colorB * (0.5 - mu) * 2.0;
  else
    return colorB * (mu - 0.5) * 2.0 + colorC * (1.0 - mu) * 2.0;
}

void main(void)
{
  // now, we will have to do the blending ourselves
  // vec4 result = mix(color0, color1, color1.a);

  // using the interpolated texture coordinate, 
  // find the color of the bottom image
  vec4 color0 = texture2D( tex0, gl_TexCoord[0].st );
  // do the same for the top image
  vec4 color1 = texture2D( tex1, gl_TexCoord[0].st );

  /*
  color1.rgb = vec3(1.0) - color1.rgb;

  gl_FragColor.rgb = color0.rgb * color1.rgb;
  gl_FragColor.a = 1.0;
  //*/

  vec4 colorA = vec4(1.0, 0.0, 0.0, 1.0);
  vec4 colorB = vec4(0.0, 1.0, 0.0, 1.0);
  vec4 colorC = vec4(0.0, 0.0, 1.0, 1.0);

  vec4 result = lerpColor(colorA, colorB, colorC, (1.0 - color1.r));
  gl_FragColor = result;
}