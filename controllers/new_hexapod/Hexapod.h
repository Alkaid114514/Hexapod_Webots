

#include <webots/Robot.hpp>
#include "Leg.h"
#include "Vector3.h"

#define COXA_LEN (38.0f/1000.0f)
#define FEMUR_LEN (79.2f/1000.0f)
#define TIBIA_LEN (116.0f/1000.0f)

#define CLIP(value, lower, upper) (((value) < (lower)) ? (lower) : ((value) > (upper) ? (upper) : (value)))


class Hexapod : public webots::Robot
{
private:
	Leg BRleg;
	Leg MRleg;
	Leg FRleg;
	Leg BLleg;
	Leg MLleg;
	Leg FLleg;

	
public:
	
	Hexapod();
	~Hexapod();

	enum GaitType
	{
		Tripod,
		Ripple,
		Wave
	};

	/// <summary>
	/// �Ի�������������Ϊԭ�������ϵΪԭ�㣬�����ȸ���������(����������ϵ)
	/// </summary>
	const Vector3 ctr2BRroot = Vector3(0.059f, -0.083f, 0.0f);
	const Vector3 ctr2MRroot = Vector3(0.08f, 0.0f, 0.0f);
	const Vector3 ctr2FRroot = Vector3(0.064f, 0.082f, 0.0f);
	const Vector3 ctr2BLroot = Vector3(-0.059f, -0.083f, 0.0f);
	const Vector3 ctr2MLroot = Vector3(-0.08f, 0.0f, 0.0f);
	const Vector3 ctr2FLroot = Vector3(-0.064f, 0.082f, 0.0f);

	/// <summary>
	/// �Ի�������������Ϊԭ�������ϵΪԭ�㣬�����ȸ���������ϵ�������������������ϵ�ļн�
	/// </summary>
	const float ctr2BRrootTheta = -(float)M_PI_4;			//-0.785398f
	const float ctr2MRrootTheta = 0.0f;
	const float ctr2FRrootTheta = (float)M_PI_4;			//0.785398f
	const float ctr2BLrootTheta = -3.0f * (float)M_PI_4;	//-2.3562f
	const float ctr2MLrootTheta = (float)M_PI;		//3.14159f
	const float ctr2FLrootTheta = 3.0f * (float)M_PI_4;	//2.3562f

	/// <summary>
	/// ����������վ��ʱ�����������������ĩ�˵�����(������ϵ)
	/// </summary>
	Vector3 initStandBR;
	Vector3 initStandMR;
	Vector3 initStandFR;
	Vector3 initStandBL;
	Vector3 initStandML;
	Vector3 initStandFL;
	float initHeight;

	Vector3 currentStandBR;
	Vector3 currentStandMR;
	Vector3 currentStandFR;
	Vector3 currentStandBL;
	Vector3 currentStandML;
	Vector3 currentStandFL;
	float currentHeight;

	/// <summary>
	/// ͬʱ�������������йؽڵĽǶȣ�������󲻻������˶�����Ҫ����startMove()������ʼ�˶�
	/// </summary>
	/// <param name="BRangles">�����ȵ������ؽڽǶ�</param>
	/// <param name="MRangles">�����ȵ������ؽڽǶ�</param>
	/// <param name="FRangles">ǰ���ȵ������ؽڽǶ�</param>
	/// <param name="BLangles">�����ȵ������ؽڽǶ�</param>
	/// <param name="MLangles">�����ȵ������ؽڽǶ�</param>
	/// <param name="FLangles">ǰ���ȵ������ؽڽǶ�</param>
	void setPose(Vector3 BRangles, Vector3 MRangles, Vector3 FRangles,
		Vector3 BLangles, Vector3 MLangles, Vector3 FLangles);
	void setBRpose(Vector3 angles);
	void setMRpose(Vector3 angles);
	void setFRpose(Vector3 angles);
	void setBLpose(Vector3 angles);
	void setMLpose(Vector3 angles);
	void setFLpose(Vector3 angles);

	/// <summary>
	/// �������˶�ѧ�����������Ҫ����Ԥ��������vector3ӦΪ�����ȸ�����Ŀ��������(���ȸ���Ϊԭ�������ϵ)
	/// </summary>
	/// <param name="vector3">�����ȸ�����Ŀ��������(������ϵ)</param>
	/// <returns>�����ؽ���ת��</returns>
	Vector3 ik(Vector3 vector3);

	/// <summary>
	/// �������˶�ѧ���������Ϊ�����ؽڵĽǶȣ������ȸ�����ĩ�˵�����(������ϵ)
	/// </summary>
	/// <param name="angles">�����ؽڵĽǶ�,xΪcoxa,yΪfemur,zΪtibia</param>
	/// <returns>�ȸ�����ĩ�˵�����(������ϵ)</returns>
	Vector3 fk(Vector3 angles);

	/// <summary>
	/// �����ȸ���Ϊԭ����������(������ϵ)ת��Ϊ�Ի�������������Ϊԭ��ָ��Ŀ��������(����������ϵ)
	/// </summary>
	/// <param name="relevant">���ȸ���Ϊԭ����������(������ϵ)</param>
	/// <param name="bias">�Ի�������������Ϊԭ��ָ���ȸ���������(����������ϵ)</param>
	/// <param name="theta">�Ի�������������Ϊԭ������ϵ�£��ȸ���Ϊԭ�������ϵ����ת��</param>
	/// <returns>�Ի�������������Ϊԭ��ָ��Ŀ��������(����������ϵ)</returns>
	Vector3 leg2bodyCoord(Vector3 relevant,Vector3 bias,float theta);

	/// <summary>
	/// ���Ի�������������Ϊԭ��ָ��Ŀ��������(����������ϵ)ת��Ϊ���ȸ���Ϊԭ����������(������ϵ)
	/// </summary>
	/// <param name="absolute">�Ի�������������Ϊԭ��ָ��Ŀ��������(����������ϵ)</param>
	/// <param name="bias">�Ի�������������Ϊԭ��ָ���ȸ���������(����������ϵ)</param>
	/// <param name="theta">�Ի�������������Ϊԭ������ϵ�£��ȸ���Ϊԭ�������ϵ����ת��</param>
	/// <returns>���ȸ���Ϊԭ����������(������ϵ)</returns>
	Vector3 body2legCoord(Vector3 absolute, Vector3 bias, float theta);

	void move(Vector3 velocity,float omega, float timeStep);

	Vector3 getNextBodyTarget(Vector3 velocity, float omega, Vector3 r0, Vector3 initial,Vector3 legBias,float timeStep);

	//Leg* getLegGroup(GaitType gaitType);

	/// <summary>
	/// ���е�����Ŀ���ĺ���������ʹ������ֱ���˶�����Ҫ������Ŀ�������startMove()������ʼ�˶�
	/// </summary>
	void setTargets(Vector3 BRtarget,Vector3 MRtarget,Vector3 FRtarget,Vector3 BLtarget,Vector3 MLtarget,Vector3 FLtarget);
	void setBRbodyTarget(Vector3 target);
	void setMRbodyTarget(Vector3 target);
	void setFRbodyTarget(Vector3 target);
	void setBLbodyTarget(Vector3 target);
	void setMLbodyTarget(Vector3 target);
	void setFLbodyTarget(Vector3 target);

	void setBRlegTarget(Vector3 target);
	void setMRlegTarget(Vector3 target);
	void setFRlegTarget(Vector3 target);
	void setBLlegTarget(Vector3 target);
	void setMLlegTarget(Vector3 target);
	void setFLlegTarget(Vector3 target);

	void setHeight(float height);

	/// <summary>
	/// ��ʼ�������õĽǶȺ�Ŀ����˶�
	/// </summary>
	void startMove();
};

