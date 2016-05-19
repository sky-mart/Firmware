python_dumper=~/Dev/MyProjects/Firmware/Tools/sdlog2/sdlog2_dump.py
echo $python_dumper

log_dir=$1

session_list=`ls $log_dir`

for session in $session_list; do
	session_full_path=$log_dir$session
	px4log_list=`ls $session_full_path | grep ".px4log"`
	echo $session
	for px4log in $px4log_list; do
		px4log_full_path=$session_full_path/$px4log
		echo $px4log_full_path
		csvlog_full_path=$session_full_path/${px4log%.px4log}.csv
		echo $csvlog_full_path

		python $python_dumper $px4log_full_path -m TIME -m OUT0 -m ATT > $csvlog_full_path
	done
done