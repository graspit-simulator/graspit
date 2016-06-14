
varying float depth;

vec3 pack3(float d)
{
	float d1;
	float d2;
	float d3;

	d1 = floor(d/(255.0*255.0));
	d = d-d1*(255.0*255.0);
	d2 = floor(d/255.0);
	d3 = d - d2*255.0;

	return vec3(d1/255.0, d2/255.0, d3/255.0);
}

void main()
{
    gl_FragColor = vec4(pack3(depth), 1);
}
