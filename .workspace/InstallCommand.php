<?php

class InstallCommand extends Command
{
    public function getName()
    {
        return 'install';
    }

    public function getDescription()
    {
        return array('Install a package');
    }

    public function getUsage()
    {
        return $this->getName().' [package]';
    }

    public function run(array $args)
    {
        $https = false;
        $default = false;
        if ($args) {
            // First : try to find flags 
            $offset = 0;
            foreach ($args as $arg) {
              if (strcmp($arg,"--default") == 0) {
                $default = true;
                $offset = $offset + 1;
              } else if (strcmp($arg,"--https") == 0) {
                $https = true;
                $offset = $offset + 1;
                echo "Enabling https\n";
              } else {
                break;
              }
            }
            $args = array_slice($args, $offset);
        }
        if ($args) {
            foreach ($args as $arg) {
                $this->workspace->install($arg, $https, $default);
            }
        } else {
            foreach ($this->workspace->getRepositories() as $repository) {
                $this->workspace->install($repository->getName(), $https, $default);
            }
        }
    }
}
