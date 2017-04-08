class  FilterBuBp1
{
	public:
		FilterBuBp1()
		{
			v[0]=0.0;
			v[1]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (5.685851241464646710e-1 * x)
				 + (0.09826663023118166473 * v[0])
				 + (0.70172780046568461465 * v[1]);
			return 
				 (v[2] - v[0]);
		}
};
