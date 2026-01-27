0. import globals and class definition
1. slrm module factory await import
2. cd module factory await import
3. SlrmModule await generation
4. CdModule await generation
5. SlrmModule return value definition generation

6. construct ik loop control object with step func (loopObject constructor)

7. onmessage handler
   7.1 init
	   7.1.1 prepare joints, endLink vectors in loopObject(prepareVectors)
	   7.1.2 construct cmdVelGen obj for cal
	   7.1.3 set cmdVelGen to loopObject(prepareCmdVelGen)
		     loopObject(prepareGjkCd) if needed
	   7.1.4 set joint limits loopObject(setJointLimits)
8.0 mainLoop function definition
8.1 self.postMessage({type: 'ready'}); mainLoop
