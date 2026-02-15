package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.paths.PathChain;

public abstract class BasePaths {
	public abstract PathChain getPreload();
	public abstract PathChain getGPP();
	public abstract PathChain getGPPReturn();
	public abstract PathChain getPGP();
	public abstract PathChain getPGPReturn();
	public abstract PathChain getPPG();
	public abstract PathChain getPPGReturn();
	public abstract PathChain getLeaveZone();
}
