How to run Amber:

1. If you have a working shell, and make, run the command 'make' from the project path '../amber/' to compile Amber for your system.
2. Amber usage:
	2a. Creating Amber database from *.nt format of RDF files
		./amber d <arg1> <arg2>
		where,
		<arg1> = path of the RDF file
		<arg2> = name of the RDF file in *.nt format (e.g., sample.nt)
	2b. Querying RDF database
		./amber q <arg1> <arg2> <arg3> <arg4> <arg5>
		where,
		<arg1> = [data path]
		<arg2> = [query path]
		<arg3> = [result path], where statistics of results are outputted
		<arg4> = [number of queries]
		<arg5> = [-yes | -no] tells Amber if the SPARQL solutions have to be printed.
